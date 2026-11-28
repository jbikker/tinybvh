# Generates external/vulkan/vk_min.h: the Vulkan declarations the given sources use, with everything they depend
# on, copied from the Khronos header (vulkan_core.h). Enums keep only the values in use. Function prototypes are
# left out: sources fetch entry points at run time (vkGetInstanceProcAddr) through the PFN_ typedefs.
# Also writes vk_min_check.cpp, which prints the size and member offsets of every struct and the value of every
# constant; compile it once with -DOFFICIAL (Vulkan SDK headers) and once without, and compare the outputs.
#
# usage: py -3 tools/gen_vk_min.py <path to vulkan_core.h> <out dir> <source files...>
# e.g.:  py -3 tools/gen_vk_min.py "%VULKAN_SDK%/Include/vulkan/vulkan_core.h" external/vulkan benchmark/vulkan_rt.cpp
import re, sys, os

TOKEN = re.compile( r'\b(?:Vk\w+|PFN_vk\w+|VK_\w+|vk[A-Z]\w*)\b' )
SKIP_DEFINES = { 'VK_DEFINE_HANDLE', 'VK_DEFINE_NON_DISPATCHABLE_HANDLE', 'VK_USE_64_BIT_PTR_DEFINES', 'VK_NULL_HANDLE' }

def parse( lines ):
	decls = {} # name -> dict( order, kind, head, body, tail ); enums keep their entries apart
	enumOf = {} # enumerator -> enum name
	i, n = 0, len( lines )
	while i < n:
		l = lines[i]
		m = re.match( r'typedef (struct|union|enum) (\w+) \{', l )
		if m:
			kind, name, body = m.group( 1 ), m.group( 2 ), []
			i += 1
			while not re.match( r'^\} (\w+);', lines[i] ): body.append( lines[i] ); i += 1
			tail = lines[i]
			if kind == 'enum':
				entries, depth = [], 0
				for b in body:
					s = b.strip()
					if s.startswith( '#if' ): depth += 1; continue
					if s.startswith( '#endif' ): depth -= 1; continue
					if s.startswith( '#' ) or s.startswith( '//' ) or not s: continue
					em = re.match( r'(\w+)\s*=\s*([^,]+),?$', s )
					if em and depth == 0: entries.append( (em.group( 1 ), em.group( 2 ).strip()) ); enumOf[em.group( 1 )] = name
				decls[name] = dict( order=i, kind='enum', head=l, entries=entries, tail=tail )
			else:
				for b in body:
					if b.strip().startswith( '#' ): sys.exit( 'preprocessor line inside %s: cannot copy safely' % name )
				decls[name] = dict( order=i, kind=kind, text='\n'.join( [l] + body + [tail] ), body=body )
			i += 1; continue
		m = re.match( r'(VK_DEFINE_HANDLE|VK_DEFINE_NON_DISPATCHABLE_HANDLE)\((\w+)\)', l )
		if m: decls[m.group( 2 )] = dict( order=i, kind='handle', text=l ); i += 1; continue
		m = re.match( r'typedef .*\(VKAPI_PTR \*(PFN_\w+)\)', l )
		if m:
			text = l
			while not text.rstrip().endswith( ';' ): i += 1; text += '\n' + lines[i]
			decls[m.group( 1 )] = dict( order=i, kind='pfn', text=text ); i += 1; continue
		m = re.match( r'typedef (\w+) (\w+);', l )
		if m: decls[m.group( 2 )] = dict( order=i, kind='typedef', text=l ); i += 1; continue
		m = re.match( r'static const \w+ (VK_\w+) = ', l )
		if m: decls[m.group( 1 )] = dict( order=i, kind='const', text=l ); i += 1; continue
		m = re.match( r'#define (VK_\w+)\b', l )
		if m and m.group( 1 ) not in SKIP_DEFINES:
			text = l
			while text.endswith( '\\' ): i += 1; text += '\n' + lines[i]
			if m.group( 1 ) not in decls: decls[m.group( 1 )] = dict( order=i, kind='define', text=text )
			i += 1; continue
		i += 1
	return decls, enumOf

def main():
	header, outDir, sources = sys.argv[1], sys.argv[2], sys.argv[3:]
	lines = open( header, encoding='utf-8' ).read().splitlines()
	decls, enumOf = parse( lines )
	wanted = set()
	for s in sources:
		for t in TOKEN.findall( open( s, encoding='utf-8' ).read() ):
			wanted.add( 'PFN_' + t if t.startswith( 'vk' ) else t )
	keep, keepEnum, todo = set(), {}, list( wanted )
	def deps_of( name ):
		d = decls[name]
		if d['kind'] == 'enum': return [v for (e, v) in d['entries'] if e in keepEnum.get( name, set() )]
		return [d['text']]
	while todo:
		t = todo.pop()
		if t in enumOf: # an enumerator: its enum, and whatever its value refers to
			e = enumOf[t]
			if t in keepEnum.setdefault( e, set() ): continue
			keepEnum[e].add( t )
			if e not in keep: keep.add( e ); todo.append( e )
			for (name, v) in decls[e]['entries']:
				if name == t: todo += TOKEN.findall( v )
			continue
		if t not in decls or t in keep and decls[t]['kind'] != 'enum': continue
		if t in keep: continue
		keep.add( t )
		for text in deps_of( t ): todo += [x for x in TOKEN.findall( text ) if x != t]
	unknownFuncs = sorted( w for w in wanted if w.startswith( 'PFN_vk' ) and w not in decls )
	if unknownFuncs: print( 'note: not in the header (ignored):', ' '.join( unknownFuncs ) )
	version = next( (l.split()[-1] for l in lines if l.startswith( '#define VK_HEADER_VERSION ' )), '?' )
	license = []
	for l in lines[2:]:
		license.append( l )
		if l.strip() == '*/': break
	out = [ '#pragma once', '// Vulkan declarations used by the TinyBVH benchmark (benchmark/vulkan_rt.cpp), copied from the Khronos',
		'// vulkan_core.h (header version %s) by tools/gen_vk_min.py. Enums hold only the values in use. Regenerate' % version,
		'// after using more of the API; check with vk_min_check.cpp.' ] + license + [
		'', '#include <stddef.h>', '#include <stdint.h>', '',
		'#if defined(_WIN32)', '#define VKAPI_ATTR', '#define VKAPI_CALL __stdcall', '#define VKAPI_PTR VKAPI_CALL',
		'#else', '#define VKAPI_ATTR', '#define VKAPI_CALL', '#define VKAPI_PTR', '#endif', '',
		'#define VK_DEFINE_HANDLE(object) typedef struct object##_T* object;',
		'#if defined(__LP64__) || defined(_WIN64) || (defined(__x86_64__) && !defined(__ILP32__)) || defined(_M_X64) || defined(__ia64) || defined(_M_IA64) || defined(__aarch64__) || defined(__powerpc64__) || (defined(__riscv) && __riscv_xlen == 64)',
		'#define VK_DEFINE_NON_DISPATCHABLE_HANDLE(object) typedef struct object##_T *object;', '#else',
		'#define VK_DEFINE_NON_DISPATCHABLE_HANDLE(object) typedef uint64_t object;', '#endif', '' ]
	check = [ '// Prints sizes, member offsets and constants of vk_min.h; compare a -DOFFICIAL build (Vulkan SDK) with a plain one.',
		'#ifdef OFFICIAL', '#include <vulkan/vulkan_core.h>', '#else', '#include "vk_min.h"', '#endif',
		'#include <cstdio>', '#include <cstddef>', 'int main()', '{' ]
	for name in sorted( keep, key=lambda k: decls[k]['order'] ):
		d = decls[name]
		if d['kind'] == 'enum':
			used = keepEnum.get( name, set() )
			entries = [(e, v) for (e, v) in d['entries'] if e in used or re.search( r'_MAX_ENUM(_[A-Z]+)?$', e )] # keep the 32-bit sentinel
			out.append( d['head'] )
			out += [ '    %s = %s,' % ev for ev in entries ]
			out.append( d['tail'] )
			check += [ '\tprintf( "%s %%lld\\n", (long long)%s );' % (e, e) for (e, v) in entries ]
		else:
			out.append( d['text'] )
			if d['kind'] in ('struct', 'union'):
				check.append( '\tprintf( "sizeof %s %%zu\\n", sizeof( %s ) );' % (name, name) )
				for b in d['body']:
					mm = re.match( r'\s*[\w\s\*]+?\b(\w+)\s*((?:\[[^\]]*\])*)\s*(:\s*\d+)?;', b )
					if mm and not mm.group( 3 ): check.append( '\tprintf( "%s.%s %%zu\\n", offsetof( %s, %s ) );' % (name, mm.group( 1 ), name, mm.group( 1 )) )
			if d['kind'] == 'define' and '(' not in d['text'].split()[1] and '"' not in d['text']:
				check.append( '\tprintf( "%s %%llu\\n", (unsigned long long)(%s) );' % (name, name) )
		out.append( '' )
	check += [ '\treturn 0;', '}', '' ]
	os.makedirs( outDir, exist_ok=True )
	open( os.path.join( outDir, 'vk_min.h' ), 'w', newline='\r\n' ).write( '\n'.join( out ) + '\n' )
	open( os.path.join( outDir, 'vk_min_check.cpp' ), 'w', newline='\r\n' ).write( '\n'.join( check ) )
	print( 'vk_min.h: %d declarations, %d lines' % (len( keep ), len( out )) )

main()
