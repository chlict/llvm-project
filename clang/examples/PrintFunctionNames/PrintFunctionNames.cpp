//===- PrintFunctionNames.cpp ---------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// Example clang plugin which simply prints the names of all the top-level decls
// in the input file.
//
//===----------------------------------------------------------------------===//

#include "clang/AST/AST.h"
#include "clang/AST/ASTConsumer.h"
#include "clang/AST/IgnoreExpr.h"
#include "clang/AST/RecursiveASTVisitor.h"
#include "clang/Frontend/CompilerInstance.h"
#include "clang/Frontend/FrontendPluginRegistry.h"
#include "clang/Sema/Sema.h"
#include "llvm/Support/raw_ostream.h"
using namespace clang;

namespace {
struct Visitor2 : public RecursiveASTVisitor<Visitor2> {
  bool VisitCallExpr(CallExpr *Call) {
    llvm::errs() << "----VisitCallExpr\n";
    Call->dump();
    if (auto FD = dyn_cast<const FunctionDecl>(Call->getCalleeDecl())) {
      if (FD->getQualifiedNameAsString() == "edsl::let") {
        return HandleLet(Call);
      }
    };
    return true;
  }

  bool HandleLet(CallExpr *Call) {
    for (Expr *Argument : Call->arguments()) {
      // Visit(Argument)
      llvm::errs() << "------Argument\n";
      Argument->dump();
      // Ignore MaterializeTemporaryExpr & ImplicitCastExpr
      auto SubExpr = IgnoreImplicitCastsExtraSingleStep(Argument);
      llvm::errs() << "----IgnoreImplicit...\n";
      SubExpr->dump();
    }
    return true;
  }
};

struct Visitor1 : public RecursiveASTVisitor<Visitor1> {
  bool VisitFunctionDecl(FunctionDecl *FD) {
    static int count = 0;
    // llvm::outs() << count << " FD->getQualifiedNameAsString() = " <<
    // FD->getQualifiedNameAsString() << "\n";
    count++;
    if (FD->getQualifiedNameAsString() == "Test3") {
      llvm::errs() << "Found Test3 - Call Visitor2\n";
      FD->dump();
      Visitor2 v2;
      v2.TraverseDecl(FD);
    }
    return true;
  }
};

class PrintFunctionsConsumer : public ASTConsumer {
  CompilerInstance &Instance;
  std::set<std::string> ParsedTemplates;

public:
  PrintFunctionsConsumer(CompilerInstance &Instance,
                         std::set<std::string> ParsedTemplates)
      : Instance(Instance), ParsedTemplates(ParsedTemplates) {}

  bool HandleTopLevelDecl(DeclGroupRef DG) override {
    llvm::errs() << "Enter HandleTopLevelDecl\n";
    // for (DeclGroupRef::iterator i = DG.begin(), e = DG.end(); i != e; ++i) {
    //   const Decl *D = *i;
    //   if (const NamedDecl *ND = dyn_cast<NamedDecl>(D))
    //     llvm::errs() << "top-level-decl: \"" << ND->getNameAsString() <<
    //     "\"\n";
    // }

    return true;
  }

  void HandleTranslationUnit(ASTContext &context) override {
    llvm::errs() << "-----Enter HandleTranslationUnit\n";
    Visitor1 visitor1;
    visitor1.TraverseDecl(context.getTranslationUnitDecl());
    // if (!Instance.getLangOpts().DelayedTemplateParsing)
    //   return;

    // This demonstrates how to force instantiation of some templates in
    // -fdelayed-template-parsing mode. (Note: Doing this unconditionally for
    // all templates is similar to not using -fdelayed-template-parsig in the
    // first place.)
    // The advantage of doing this in HandleTranslationUnit() is that all
    // codegen (when using -add-plugin) is completely finished and this can't
    // affect the compiler output.
    struct Visitor : public RecursiveASTVisitor<Visitor> {
      const std::set<std::string> &ParsedTemplates;
      Visitor(const std::set<std::string> &ParsedTemplates)
          : ParsedTemplates(ParsedTemplates) {}
      bool VisitFunctionDecl(FunctionDecl *FD) {
        if (FD->isLateTemplateParsed() &&
            ParsedTemplates.count(FD->getNameAsString()))
          LateParsedDecls.insert(FD);
        return true;
      }

      std::set<FunctionDecl*> LateParsedDecls;
    } v(ParsedTemplates);
    v.TraverseDecl(context.getTranslationUnitDecl());
    clang::Sema &sema = Instance.getSema();
    for (const FunctionDecl *FD : v.LateParsedDecls) {
      clang::LateParsedTemplate &LPT =
          *sema.LateParsedTemplateMap.find(FD)->second;
      sema.LateTemplateParser(sema.OpaqueParser, LPT);
      llvm::errs() << "late-parsed-decl: \"" << FD->getNameAsString() << "\"\n";
    }
  }
};

class PrintFunctionNamesAction : public PluginASTAction {
  std::set<std::string> ParsedTemplates;

protected:
  std::unique_ptr<ASTConsumer> CreateASTConsumer(CompilerInstance &CI,
                                                 llvm::StringRef) override {
    return std::make_unique<PrintFunctionsConsumer>(CI, ParsedTemplates);
  }

  bool ParseArgs(const CompilerInstance &CI,
                 const std::vector<std::string> &args) override {
    for (unsigned i = 0, e = args.size(); i != e; ++i) {
      llvm::errs() << "PrintFunctionNames arg = " << args[i] << "\n";

      // Example error handling.
      DiagnosticsEngine &D = CI.getDiagnostics();
      if (args[i] == "-an-error") {
        unsigned DiagID = D.getCustomDiagID(DiagnosticsEngine::Error,
                                            "invalid argument '%0'");
        D.Report(DiagID) << args[i];
        return false;
      } else if (args[i] == "-parse-template") {
        if (i + 1 >= e) {
          D.Report(D.getCustomDiagID(DiagnosticsEngine::Error,
                                     "missing -parse-template argument"));
          return false;
        }
        ++i;
        ParsedTemplates.insert(args[i]);
      }
    }
    if (!args.empty() && args[0] == "help")
      PrintHelp(llvm::errs());

    return true;
  }
  void PrintHelp(llvm::raw_ostream& ros) {
    ros << "Help for PrintFunctionNames plugin goes here\n";
  }

};

}

static FrontendPluginRegistry::Add<PrintFunctionNamesAction>
X("print-fns", "print function names");
